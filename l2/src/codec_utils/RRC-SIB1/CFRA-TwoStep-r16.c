/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "CFRA-TwoStep-r16.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static int
memb_ssb_ResourceList_constraint_25(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	/* Determine the number of elements */
	size = _A_CSEQUENCE_FROM_VOID(sptr)->count;
	
	if((size >= 1UL && size <= 64UL)) {
		/* Perform validation of the inner elements */
		return SEQUENCE_OF_constraint(td, sptr, ctfailcb, app_key);
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_ra_ssb_OccasionMaskIndex_constraint_25(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0L && value <= 15L)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_ssb_PerRACH_OccasionTwoStepRA_r16_constr_4 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (0..7) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_msgA_TransMax_r16_constr_14 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  9 }	/* (0..9) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_ssb_ResourceList_constr_26 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 6,  6,  1,  64 }	/* (SIZE(1..64)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_ssb_ResourceList_constr_26 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 6,  6,  1,  64 }	/* (SIZE(1..64)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_ra_ssb_OccasionMaskIndex_constr_28 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  15 }	/* (0..15) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_ssb_PerRACH_OccasionTwoStepRA_r16_value2enum_4[] = {
	{ 0,	9,	"oneEighth" },
	{ 1,	9,	"oneFourth" },
	{ 2,	7,	"oneHalf" },
	{ 3,	3,	"one" },
	{ 4,	3,	"two" },
	{ 5,	4,	"four" },
	{ 6,	5,	"eight" },
	{ 7,	7,	"sixteen" }
};
static const unsigned int asn_MAP_ssb_PerRACH_OccasionTwoStepRA_r16_enum2value_4[] = {
	6,	/* eight(6) */
	5,	/* four(5) */
	3,	/* one(3) */
	0,	/* oneEighth(0) */
	1,	/* oneFourth(1) */
	2,	/* oneHalf(2) */
	7,	/* sixteen(7) */
	4	/* two(4) */
};
static const asn_INTEGER_specifics_t asn_SPC_ssb_PerRACH_OccasionTwoStepRA_r16_specs_4 = {
	asn_MAP_ssb_PerRACH_OccasionTwoStepRA_r16_value2enum_4,	/* "tag" => N; sorted by tag */
	asn_MAP_ssb_PerRACH_OccasionTwoStepRA_r16_enum2value_4,	/* N => "tag"; sorted by N */
	8,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_ssb_PerRACH_OccasionTwoStepRA_r16_tags_4[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_ssb_PerRACH_OccasionTwoStepRA_r16_4 = {
	"ssb-PerRACH-OccasionTwoStepRA-r16",
	"ssb-PerRACH-OccasionTwoStepRA-r16",
	&asn_OP_NativeEnumerated,
	asn_DEF_ssb_PerRACH_OccasionTwoStepRA_r16_tags_4,
	sizeof(asn_DEF_ssb_PerRACH_OccasionTwoStepRA_r16_tags_4)
		/sizeof(asn_DEF_ssb_PerRACH_OccasionTwoStepRA_r16_tags_4[0]) - 1, /* 1 */
	asn_DEF_ssb_PerRACH_OccasionTwoStepRA_r16_tags_4,	/* Same as above */
	sizeof(asn_DEF_ssb_PerRACH_OccasionTwoStepRA_r16_tags_4)
		/sizeof(asn_DEF_ssb_PerRACH_OccasionTwoStepRA_r16_tags_4[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_ssb_PerRACH_OccasionTwoStepRA_r16_constr_4,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_ssb_PerRACH_OccasionTwoStepRA_r16_specs_4	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_occasionsTwoStepRA_r16_2[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct CFRA_TwoStep_r16__occasionsTwoStepRA_r16, rach_ConfigGenericTwoStepRA_r16),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RACH_ConfigGenericTwoStepRA_r16,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"rach-ConfigGenericTwoStepRA-r16"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct CFRA_TwoStep_r16__occasionsTwoStepRA_r16, ssb_PerRACH_OccasionTwoStepRA_r16),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ssb_PerRACH_OccasionTwoStepRA_r16_4,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"ssb-PerRACH-OccasionTwoStepRA-r16"
		},
};
static const ber_tlv_tag_t asn_DEF_occasionsTwoStepRA_r16_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_occasionsTwoStepRA_r16_tag2el_2[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* rach-ConfigGenericTwoStepRA-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* ssb-PerRACH-OccasionTwoStepRA-r16 */
};
static asn_SEQUENCE_specifics_t asn_SPC_occasionsTwoStepRA_r16_specs_2 = {
	sizeof(struct CFRA_TwoStep_r16__occasionsTwoStepRA_r16),
	offsetof(struct CFRA_TwoStep_r16__occasionsTwoStepRA_r16, _asn_ctx),
	asn_MAP_occasionsTwoStepRA_r16_tag2el_2,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_occasionsTwoStepRA_r16_2 = {
	"occasionsTwoStepRA-r16",
	"occasionsTwoStepRA-r16",
	&asn_OP_SEQUENCE,
	asn_DEF_occasionsTwoStepRA_r16_tags_2,
	sizeof(asn_DEF_occasionsTwoStepRA_r16_tags_2)
		/sizeof(asn_DEF_occasionsTwoStepRA_r16_tags_2[0]) - 1, /* 1 */
	asn_DEF_occasionsTwoStepRA_r16_tags_2,	/* Same as above */
	sizeof(asn_DEF_occasionsTwoStepRA_r16_tags_2)
		/sizeof(asn_DEF_occasionsTwoStepRA_r16_tags_2[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_occasionsTwoStepRA_r16_2,
	2,	/* Elements count */
	&asn_SPC_occasionsTwoStepRA_r16_specs_2	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_msgA_TransMax_r16_value2enum_14[] = {
	{ 0,	2,	"n1" },
	{ 1,	2,	"n2" },
	{ 2,	2,	"n4" },
	{ 3,	2,	"n6" },
	{ 4,	2,	"n8" },
	{ 5,	3,	"n10" },
	{ 6,	3,	"n20" },
	{ 7,	3,	"n50" },
	{ 8,	4,	"n100" },
	{ 9,	4,	"n200" }
};
static const unsigned int asn_MAP_msgA_TransMax_r16_enum2value_14[] = {
	0,	/* n1(0) */
	5,	/* n10(5) */
	8,	/* n100(8) */
	1,	/* n2(1) */
	6,	/* n20(6) */
	9,	/* n200(9) */
	2,	/* n4(2) */
	7,	/* n50(7) */
	3,	/* n6(3) */
	4	/* n8(4) */
};
static const asn_INTEGER_specifics_t asn_SPC_msgA_TransMax_r16_specs_14 = {
	asn_MAP_msgA_TransMax_r16_value2enum_14,	/* "tag" => N; sorted by tag */
	asn_MAP_msgA_TransMax_r16_enum2value_14,	/* N => "tag"; sorted by N */
	10,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_msgA_TransMax_r16_tags_14[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_msgA_TransMax_r16_14 = {
	"msgA-TransMax-r16",
	"msgA-TransMax-r16",
	&asn_OP_NativeEnumerated,
	asn_DEF_msgA_TransMax_r16_tags_14,
	sizeof(asn_DEF_msgA_TransMax_r16_tags_14)
		/sizeof(asn_DEF_msgA_TransMax_r16_tags_14[0]) - 1, /* 1 */
	asn_DEF_msgA_TransMax_r16_tags_14,	/* Same as above */
	sizeof(asn_DEF_msgA_TransMax_r16_tags_14)
		/sizeof(asn_DEF_msgA_TransMax_r16_tags_14[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_msgA_TransMax_r16_constr_14,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_msgA_TransMax_r16_specs_14	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_ssb_ResourceList_26[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_CFRA_SSB_Resource,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_ssb_ResourceList_tags_26[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_ssb_ResourceList_specs_26 = {
	sizeof(struct CFRA_TwoStep_r16__resourcesTwoStep_r16__ssb_ResourceList),
	offsetof(struct CFRA_TwoStep_r16__resourcesTwoStep_r16__ssb_ResourceList, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_ssb_ResourceList_26 = {
	"ssb-ResourceList",
	"ssb-ResourceList",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_ssb_ResourceList_tags_26,
	sizeof(asn_DEF_ssb_ResourceList_tags_26)
		/sizeof(asn_DEF_ssb_ResourceList_tags_26[0]) - 1, /* 1 */
	asn_DEF_ssb_ResourceList_tags_26,	/* Same as above */
	sizeof(asn_DEF_ssb_ResourceList_tags_26)
		/sizeof(asn_DEF_ssb_ResourceList_tags_26[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_ssb_ResourceList_constr_26,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_OF_constraint
	},
	asn_MBR_ssb_ResourceList_26,
	1,	/* Single element */
	&asn_SPC_ssb_ResourceList_specs_26	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_resourcesTwoStep_r16_25[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct CFRA_TwoStep_r16__resourcesTwoStep_r16, ssb_ResourceList),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		0,
		&asn_DEF_ssb_ResourceList_26,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_ssb_ResourceList_constr_26,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_ssb_ResourceList_constraint_25
		},
		0, 0, /* No default value */
		"ssb-ResourceList"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct CFRA_TwoStep_r16__resourcesTwoStep_r16, ra_ssb_OccasionMaskIndex),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_ra_ssb_OccasionMaskIndex_constr_28,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_ra_ssb_OccasionMaskIndex_constraint_25
		},
		0, 0, /* No default value */
		"ra-ssb-OccasionMaskIndex"
		},
};
static const ber_tlv_tag_t asn_DEF_resourcesTwoStep_r16_tags_25[] = {
	(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_resourcesTwoStep_r16_tag2el_25[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* ssb-ResourceList */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* ra-ssb-OccasionMaskIndex */
};
static asn_SEQUENCE_specifics_t asn_SPC_resourcesTwoStep_r16_specs_25 = {
	sizeof(struct CFRA_TwoStep_r16__resourcesTwoStep_r16),
	offsetof(struct CFRA_TwoStep_r16__resourcesTwoStep_r16, _asn_ctx),
	asn_MAP_resourcesTwoStep_r16_tag2el_25,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_resourcesTwoStep_r16_25 = {
	"resourcesTwoStep-r16",
	"resourcesTwoStep-r16",
	&asn_OP_SEQUENCE,
	asn_DEF_resourcesTwoStep_r16_tags_25,
	sizeof(asn_DEF_resourcesTwoStep_r16_tags_25)
		/sizeof(asn_DEF_resourcesTwoStep_r16_tags_25[0]) - 1, /* 1 */
	asn_DEF_resourcesTwoStep_r16_tags_25,	/* Same as above */
	sizeof(asn_DEF_resourcesTwoStep_r16_tags_25)
		/sizeof(asn_DEF_resourcesTwoStep_r16_tags_25[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_resourcesTwoStep_r16_25,
	2,	/* Elements count */
	&asn_SPC_resourcesTwoStep_r16_specs_25	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_CFRA_TwoStep_r16_1[] = {
	{ ATF_POINTER, 1, offsetof(struct CFRA_TwoStep_r16, occasionsTwoStepRA_r16),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		0,
		&asn_DEF_occasionsTwoStepRA_r16_2,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"occasionsTwoStepRA-r16"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct CFRA_TwoStep_r16, msgA_CFRA_PUSCH_r16),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_MsgA_PUSCH_Resource_r16,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"msgA-CFRA-PUSCH-r16"
		},
	{ ATF_POINTER, 1, offsetof(struct CFRA_TwoStep_r16, msgA_TransMax_r16),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_msgA_TransMax_r16_14,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"msgA-TransMax-r16"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct CFRA_TwoStep_r16, resourcesTwoStep_r16),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		0,
		&asn_DEF_resourcesTwoStep_r16_25,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"resourcesTwoStep-r16"
		},
};
static const int asn_MAP_CFRA_TwoStep_r16_oms_1[] = { 0, 2 };
static const ber_tlv_tag_t asn_DEF_CFRA_TwoStep_r16_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_CFRA_TwoStep_r16_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* occasionsTwoStepRA-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* msgA-CFRA-PUSCH-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* msgA-TransMax-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* resourcesTwoStep-r16 */
};
asn_SEQUENCE_specifics_t asn_SPC_CFRA_TwoStep_r16_specs_1 = {
	sizeof(struct CFRA_TwoStep_r16),
	offsetof(struct CFRA_TwoStep_r16, _asn_ctx),
	asn_MAP_CFRA_TwoStep_r16_tag2el_1,
	4,	/* Count of tags in the map */
	asn_MAP_CFRA_TwoStep_r16_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	4,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_CFRA_TwoStep_r16 = {
	"CFRA-TwoStep-r16",
	"CFRA-TwoStep-r16",
	&asn_OP_SEQUENCE,
	asn_DEF_CFRA_TwoStep_r16_tags_1,
	sizeof(asn_DEF_CFRA_TwoStep_r16_tags_1)
		/sizeof(asn_DEF_CFRA_TwoStep_r16_tags_1[0]), /* 1 */
	asn_DEF_CFRA_TwoStep_r16_tags_1,	/* Same as above */
	sizeof(asn_DEF_CFRA_TwoStep_r16_tags_1)
		/sizeof(asn_DEF_CFRA_TwoStep_r16_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_CFRA_TwoStep_r16_1,
	4,	/* Elements count */
	&asn_SPC_CFRA_TwoStep_r16_specs_1	/* Additional specs */
};

