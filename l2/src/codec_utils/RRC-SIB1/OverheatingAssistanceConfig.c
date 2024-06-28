/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "OverheatingAssistanceConfig.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_overheatingIndicationProhibitTimer_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  15 }	/* (0..15) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_overheatingIndicationProhibitTimer_value2enum_2[] = {
	{ 0,	2,	"s0" },
	{ 1,	6,	"s0dot5" },
	{ 2,	2,	"s1" },
	{ 3,	2,	"s2" },
	{ 4,	2,	"s5" },
	{ 5,	3,	"s10" },
	{ 6,	3,	"s20" },
	{ 7,	3,	"s30" },
	{ 8,	3,	"s60" },
	{ 9,	3,	"s90" },
	{ 10,	4,	"s120" },
	{ 11,	4,	"s300" },
	{ 12,	4,	"s600" },
	{ 13,	6,	"spare3" },
	{ 14,	6,	"spare2" },
	{ 15,	6,	"spare1" }
};
static const unsigned int asn_MAP_overheatingIndicationProhibitTimer_enum2value_2[] = {
	0,	/* s0(0) */
	1,	/* s0dot5(1) */
	2,	/* s1(2) */
	5,	/* s10(5) */
	10,	/* s120(10) */
	3,	/* s2(3) */
	6,	/* s20(6) */
	7,	/* s30(7) */
	11,	/* s300(11) */
	4,	/* s5(4) */
	8,	/* s60(8) */
	12,	/* s600(12) */
	9,	/* s90(9) */
	15,	/* spare1(15) */
	14,	/* spare2(14) */
	13	/* spare3(13) */
};
static const asn_INTEGER_specifics_t asn_SPC_overheatingIndicationProhibitTimer_specs_2 = {
	asn_MAP_overheatingIndicationProhibitTimer_value2enum_2,	/* "tag" => N; sorted by tag */
	asn_MAP_overheatingIndicationProhibitTimer_enum2value_2,	/* N => "tag"; sorted by N */
	16,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_overheatingIndicationProhibitTimer_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_overheatingIndicationProhibitTimer_2 = {
	"overheatingIndicationProhibitTimer",
	"overheatingIndicationProhibitTimer",
	&asn_OP_NativeEnumerated,
	asn_DEF_overheatingIndicationProhibitTimer_tags_2,
	sizeof(asn_DEF_overheatingIndicationProhibitTimer_tags_2)
		/sizeof(asn_DEF_overheatingIndicationProhibitTimer_tags_2[0]) - 1, /* 1 */
	asn_DEF_overheatingIndicationProhibitTimer_tags_2,	/* Same as above */
	sizeof(asn_DEF_overheatingIndicationProhibitTimer_tags_2)
		/sizeof(asn_DEF_overheatingIndicationProhibitTimer_tags_2[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_overheatingIndicationProhibitTimer_constr_2,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_overheatingIndicationProhibitTimer_specs_2	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_OverheatingAssistanceConfig_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct OverheatingAssistanceConfig, overheatingIndicationProhibitTimer),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_overheatingIndicationProhibitTimer_2,
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
		"overheatingIndicationProhibitTimer"
		},
};
static const ber_tlv_tag_t asn_DEF_OverheatingAssistanceConfig_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_OverheatingAssistanceConfig_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 } /* overheatingIndicationProhibitTimer */
};
asn_SEQUENCE_specifics_t asn_SPC_OverheatingAssistanceConfig_specs_1 = {
	sizeof(struct OverheatingAssistanceConfig),
	offsetof(struct OverheatingAssistanceConfig, _asn_ctx),
	asn_MAP_OverheatingAssistanceConfig_tag2el_1,
	1,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_OverheatingAssistanceConfig = {
	"OverheatingAssistanceConfig",
	"OverheatingAssistanceConfig",
	&asn_OP_SEQUENCE,
	asn_DEF_OverheatingAssistanceConfig_tags_1,
	sizeof(asn_DEF_OverheatingAssistanceConfig_tags_1)
		/sizeof(asn_DEF_OverheatingAssistanceConfig_tags_1[0]), /* 1 */
	asn_DEF_OverheatingAssistanceConfig_tags_1,	/* Same as above */
	sizeof(asn_DEF_OverheatingAssistanceConfig_tags_1)
		/sizeof(asn_DEF_OverheatingAssistanceConfig_tags_1[0]), /* 1 */
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
	asn_MBR_OverheatingAssistanceConfig_1,
	1,	/* Elements count */
	&asn_SPC_OverheatingAssistanceConfig_specs_1	/* Additional specs */
};

