/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "IAB-ResourceConfig-r17.h"

static int
memb_NativeInteger_constraint_3(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0L && value <= 5119L)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static int
memb_slotList_r17_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
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
	
	if((size >= 1UL && size <= 5120UL)) {
		/* Perform validation of the inner elements */
		return SEQUENCE_OF_constraint(td, sptr, ctfailcb, app_key);
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_Member_constr_4 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 13,  13,  0,  5119 }	/* (0..5119) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_slotList_r17_constr_3 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 13,  13,  1,  5120 }	/* (SIZE(1..5120)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_periodicitySlotList_r17_constr_5 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  11 }	/* (0..11) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_slotList_r17_constr_3 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 13,  13,  1,  5120 }	/* (SIZE(1..5120)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static asn_TYPE_member_t asn_MBR_slotList_r17_3[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (2 << 2)),
		0,
		&asn_DEF_NativeInteger,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_Member_constr_4,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_NativeInteger_constraint_3
		},
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_slotList_r17_tags_3[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_slotList_r17_specs_3 = {
	sizeof(struct IAB_ResourceConfig_r17__slotList_r17),
	offsetof(struct IAB_ResourceConfig_r17__slotList_r17, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_slotList_r17_3 = {
	"slotList-r17",
	"slotList-r17",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_slotList_r17_tags_3,
	sizeof(asn_DEF_slotList_r17_tags_3)
		/sizeof(asn_DEF_slotList_r17_tags_3[0]) - 1, /* 1 */
	asn_DEF_slotList_r17_tags_3,	/* Same as above */
	sizeof(asn_DEF_slotList_r17_tags_3)
		/sizeof(asn_DEF_slotList_r17_tags_3[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_slotList_r17_constr_3,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_OF_constraint
	},
	asn_MBR_slotList_r17_3,
	1,	/* Single element */
	&asn_SPC_slotList_r17_specs_3	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_periodicitySlotList_r17_value2enum_5[] = {
	{ 0,	5,	"ms0p5" },
	{ 1,	7,	"ms0p625" },
	{ 2,	3,	"ms1" },
	{ 3,	6,	"ms1p25" },
	{ 4,	3,	"ms2" },
	{ 5,	5,	"ms2p5" },
	{ 6,	3,	"ms5" },
	{ 7,	4,	"ms10" },
	{ 8,	4,	"ms20" },
	{ 9,	4,	"ms40" },
	{ 10,	4,	"ms80" },
	{ 11,	5,	"ms160" }
};
static const unsigned int asn_MAP_periodicitySlotList_r17_enum2value_5[] = {
	0,	/* ms0p5(0) */
	1,	/* ms0p625(1) */
	2,	/* ms1(2) */
	7,	/* ms10(7) */
	11,	/* ms160(11) */
	3,	/* ms1p25(3) */
	4,	/* ms2(4) */
	8,	/* ms20(8) */
	5,	/* ms2p5(5) */
	9,	/* ms40(9) */
	6,	/* ms5(6) */
	10	/* ms80(10) */
};
static const asn_INTEGER_specifics_t asn_SPC_periodicitySlotList_r17_specs_5 = {
	asn_MAP_periodicitySlotList_r17_value2enum_5,	/* "tag" => N; sorted by tag */
	asn_MAP_periodicitySlotList_r17_enum2value_5,	/* N => "tag"; sorted by N */
	12,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_periodicitySlotList_r17_tags_5[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_periodicitySlotList_r17_5 = {
	"periodicitySlotList-r17",
	"periodicitySlotList-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_periodicitySlotList_r17_tags_5,
	sizeof(asn_DEF_periodicitySlotList_r17_tags_5)
		/sizeof(asn_DEF_periodicitySlotList_r17_tags_5[0]) - 1, /* 1 */
	asn_DEF_periodicitySlotList_r17_tags_5,	/* Same as above */
	sizeof(asn_DEF_periodicitySlotList_r17_tags_5)
		/sizeof(asn_DEF_periodicitySlotList_r17_tags_5[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_periodicitySlotList_r17_constr_5,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_periodicitySlotList_r17_specs_5	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_IAB_ResourceConfig_r17_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct IAB_ResourceConfig_r17, iab_ResourceConfigID_r17),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_IAB_ResourceConfigID_r17,
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
		"iab-ResourceConfigID-r17"
		},
	{ ATF_POINTER, 3, offsetof(struct IAB_ResourceConfig_r17, slotList_r17),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		0,
		&asn_DEF_slotList_r17_3,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_slotList_r17_constr_3,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_slotList_r17_constraint_1
		},
		0, 0, /* No default value */
		"slotList-r17"
		},
	{ ATF_POINTER, 2, offsetof(struct IAB_ResourceConfig_r17, periodicitySlotList_r17),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_periodicitySlotList_r17_5,
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
		"periodicitySlotList-r17"
		},
	{ ATF_POINTER, 1, offsetof(struct IAB_ResourceConfig_r17, slotListSubcarrierSpacing_r17),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SubcarrierSpacing,
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
		"slotListSubcarrierSpacing-r17"
		},
};
static const int asn_MAP_IAB_ResourceConfig_r17_oms_1[] = { 1, 2, 3 };
static const ber_tlv_tag_t asn_DEF_IAB_ResourceConfig_r17_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_IAB_ResourceConfig_r17_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* iab-ResourceConfigID-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* slotList-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* periodicitySlotList-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* slotListSubcarrierSpacing-r17 */
};
asn_SEQUENCE_specifics_t asn_SPC_IAB_ResourceConfig_r17_specs_1 = {
	sizeof(struct IAB_ResourceConfig_r17),
	offsetof(struct IAB_ResourceConfig_r17, _asn_ctx),
	asn_MAP_IAB_ResourceConfig_r17_tag2el_1,
	4,	/* Count of tags in the map */
	asn_MAP_IAB_ResourceConfig_r17_oms_1,	/* Optional members */
	3, 0,	/* Root/Additions */
	4,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_IAB_ResourceConfig_r17 = {
	"IAB-ResourceConfig-r17",
	"IAB-ResourceConfig-r17",
	&asn_OP_SEQUENCE,
	asn_DEF_IAB_ResourceConfig_r17_tags_1,
	sizeof(asn_DEF_IAB_ResourceConfig_r17_tags_1)
		/sizeof(asn_DEF_IAB_ResourceConfig_r17_tags_1[0]), /* 1 */
	asn_DEF_IAB_ResourceConfig_r17_tags_1,	/* Same as above */
	sizeof(asn_DEF_IAB_ResourceConfig_r17_tags_1)
		/sizeof(asn_DEF_IAB_ResourceConfig_r17_tags_1[0]), /* 1 */
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
	asn_MBR_IAB_ResourceConfig_r17_1,
	4,	/* Elements count */
	&asn_SPC_IAB_ResourceConfig_r17_specs_1	/* Additional specs */
};

