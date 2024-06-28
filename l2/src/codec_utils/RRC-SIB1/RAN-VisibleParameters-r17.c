/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "RAN-VisibleParameters-r17.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static int
memb_numberOfBufferLevelEntries_r17_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 1L && value <= 8L)) {
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
static asn_per_constraints_t asn_PER_type_ran_VisiblePeriodicity_r17_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  4 }	/* (0..4) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_numberOfBufferLevelEntries_r17_constr_8 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  1,  8 }	/* (1..8) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_ran_VisiblePeriodicity_r17_value2enum_2[] = {
	{ 0,	5,	"ms120" },
	{ 1,	5,	"ms240" },
	{ 2,	5,	"ms480" },
	{ 3,	5,	"ms640" },
	{ 4,	6,	"ms1024" }
};
static const unsigned int asn_MAP_ran_VisiblePeriodicity_r17_enum2value_2[] = {
	4,	/* ms1024(4) */
	0,	/* ms120(0) */
	1,	/* ms240(1) */
	2,	/* ms480(2) */
	3	/* ms640(3) */
};
static const asn_INTEGER_specifics_t asn_SPC_ran_VisiblePeriodicity_r17_specs_2 = {
	asn_MAP_ran_VisiblePeriodicity_r17_value2enum_2,	/* "tag" => N; sorted by tag */
	asn_MAP_ran_VisiblePeriodicity_r17_enum2value_2,	/* N => "tag"; sorted by N */
	5,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_ran_VisiblePeriodicity_r17_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_ran_VisiblePeriodicity_r17_2 = {
	"ran-VisiblePeriodicity-r17",
	"ran-VisiblePeriodicity-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_ran_VisiblePeriodicity_r17_tags_2,
	sizeof(asn_DEF_ran_VisiblePeriodicity_r17_tags_2)
		/sizeof(asn_DEF_ran_VisiblePeriodicity_r17_tags_2[0]) - 1, /* 1 */
	asn_DEF_ran_VisiblePeriodicity_r17_tags_2,	/* Same as above */
	sizeof(asn_DEF_ran_VisiblePeriodicity_r17_tags_2)
		/sizeof(asn_DEF_ran_VisiblePeriodicity_r17_tags_2[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_ran_VisiblePeriodicity_r17_constr_2,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_ran_VisiblePeriodicity_r17_specs_2	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_RAN_VisibleParameters_r17_1[] = {
	{ ATF_POINTER, 3, offsetof(struct RAN_VisibleParameters_r17, ran_VisiblePeriodicity_r17),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ran_VisiblePeriodicity_r17_2,
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
		"ran-VisiblePeriodicity-r17"
		},
	{ ATF_POINTER, 2, offsetof(struct RAN_VisibleParameters_r17, numberOfBufferLevelEntries_r17),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_numberOfBufferLevelEntries_r17_constr_8,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_numberOfBufferLevelEntries_r17_constraint_1
		},
		0, 0, /* No default value */
		"numberOfBufferLevelEntries-r17"
		},
	{ ATF_POINTER, 1, offsetof(struct RAN_VisibleParameters_r17, reportPlayoutDelayForMediaStartup_r17),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BOOLEAN,
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
		"reportPlayoutDelayForMediaStartup-r17"
		},
};
static const int asn_MAP_RAN_VisibleParameters_r17_oms_1[] = { 0, 1, 2 };
static const ber_tlv_tag_t asn_DEF_RAN_VisibleParameters_r17_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_RAN_VisibleParameters_r17_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* ran-VisiblePeriodicity-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* numberOfBufferLevelEntries-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* reportPlayoutDelayForMediaStartup-r17 */
};
asn_SEQUENCE_specifics_t asn_SPC_RAN_VisibleParameters_r17_specs_1 = {
	sizeof(struct RAN_VisibleParameters_r17),
	offsetof(struct RAN_VisibleParameters_r17, _asn_ctx),
	asn_MAP_RAN_VisibleParameters_r17_tag2el_1,
	3,	/* Count of tags in the map */
	asn_MAP_RAN_VisibleParameters_r17_oms_1,	/* Optional members */
	3, 0,	/* Root/Additions */
	3,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_RAN_VisibleParameters_r17 = {
	"RAN-VisibleParameters-r17",
	"RAN-VisibleParameters-r17",
	&asn_OP_SEQUENCE,
	asn_DEF_RAN_VisibleParameters_r17_tags_1,
	sizeof(asn_DEF_RAN_VisibleParameters_r17_tags_1)
		/sizeof(asn_DEF_RAN_VisibleParameters_r17_tags_1[0]), /* 1 */
	asn_DEF_RAN_VisibleParameters_r17_tags_1,	/* Same as above */
	sizeof(asn_DEF_RAN_VisibleParameters_r17_tags_1)
		/sizeof(asn_DEF_RAN_VisibleParameters_r17_tags_1[0]), /* 1 */
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
	asn_MBR_RAN_VisibleParameters_r17_1,
	3,	/* Elements count */
	&asn_SPC_RAN_VisibleParameters_r17_specs_1	/* Additional specs */
};

